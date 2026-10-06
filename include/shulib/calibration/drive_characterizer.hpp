#pragma once
//
// DriveCharacterizer — shulib Calibrate's GAINS step: the robot drives itself through four short,
// bounded segments while each drive side's kS, kV and kA are fitted on the brain (chunk R5a Part A,
// 2026-09-29 — the roadmap's R5 ramp routine and fit, moved onto the robot by the team lead's ruling
// of 2026-09-26: gains are measured by the robot and kept on the SD card, never typed or committed).
//
// ── PROVEN ON THE HOST TEST PLANT; IT HAS NEVER DRIVEN A ROBOT ──────────────────────
// Every property below is shown against the A2 host test plant, whose motor model IS the law being
// fitted (sim/motor_model.hpp's honesty boundary) — so recovering the plant's gains proves the
// ARITHMETIC and every safety rule, not that a V5 drive side follows the law (A4 register HA-171).
// The plant models neither a V5 Coast nor a V5 Brake at 0 V (only its τ = kA/kV decay), so how far
// a real robot rolls or brakes after a segment is unknown until S2 (HA-53, HA-143). Every number in
// CharacterizerConfig is INVENTED (HA-170); the rest floors it shares with the drivetrain calibrator
// keep their HA-136. No program runs this yet: Part C of R5a wires it into shulib Calibrate.
//
// ── The phases ──────────────────────────────────────────────────────────────────────
//   Idle → WaitImu → [Rest → Driving → Stopping] × 4 → Done | Aborted
//   WaitImu   unpowered   wait for imu.isReady(), bounded.
//   Rest      unpowered   the drivetrain calibrator's rest definition — every member under restFloor
//                         AND the IMU's yaw rate under restYawRateFloor — held restSettleTime, bounded
//                         by restTimeout. Then every member's temperature must be UNDER tempGuard,
//                         else TooHot, naming the port and the temperature, and nothing is powered.
//   Driving   POWERED     one segment. BOTH sides get the SAME volts every tick (each member
//                         sign × volts):
//                           QuasiForward   a ramp of rampRate V/s from 0
//                           QuasiBackward  the same, backward
//                           StepForward    stepVolts held
//                           StepBackward   the same, backward — a hard reverse from rest, the
//                                          condition of 2026-09-25's IMU episode (HA-160)
//                         It ENDS — normally, never a refusal — on the FIRST of: either side's travel
//                         reaching segmentDistance (judged PER SIDE, never on the mean: the faster side
//                         is the one that runs out of room); the ramp reaching rampMax; segmentTimeout.
//   Stopping  0 V + BRAKE the end tick commands 0 V + Brake on every member — which reaches the motor
//                         since DEFECTS2 (HA-142), and which HA-53 claims is prompt — and holds it,
//                         re-commanded every tick, until the rest definition holds; then 0 V + Coast.
//                         Each side's STOPPING DISTANCE (encoders, from the end tick to rest) is
//                         measured and reported; one longer than stopBound ends the run as StopTooLong
//                         before any further segment (R5a's decision D11).
//   Done      result() is the characterization; report() every number behind it.
// The two STEP segments exist because a ramp alone cannot identify the law: once its start transient
// decays, speed is affine in volts and the fit's columns are collinear (feedforward_fit.hpp; R5a's
// re-check R2 and decision D10).
//
// ── The rules, each with the mistake it prevents ───────────────────────────────────
//  1. ONE PLACE FOR A SIGN. Each member's `sign` multiplies its command and its readings HERE, once.
//     shulib Calibrate's adapters carry POSITIVE ports (drivetrain_calibrator.hpp's rule 1); a
//     program whose adapters are already signed passes +1. Applying it twice is a five-against-four
//     fight on the robot and invisible in a symmetric test (R5a's landmine 6).
//  2. THE MEDIAN, NOT THE MEAN, for a side's speed and travel — hal::medianInPlace, the statistic
//     hal::MotorGroup reports, so a slipping member (robot two's port 18, HA-130) cannot drag a side.
//  3. THE CURVE GUARD READS THE ENCODERS, NEVER THE IMU (R5a's decision D9). Equal volts on sides whose
//     gains differ curve the robot by about segmentDistance × asymmetry / W. The guard is the
//     encoders' ψ = (dR − dL)/W from each side's median travel, refused (CurveExceeded) past
//     ψ_g = segmentDistance × asymRefuse / W, computed at construction from the DESCRIPTION's
//     effective track width. asymRefuse (0.30) is twice the 15 % asymmetry warning, so a robot that
//     warns still finishes. The IMU's accumulated yaw is RECORDED beside ψ for every segment — their
//     disagreement is HA-160 evidence — and never refuses: an IMU that jumps 37° in a tick (as on
//     2026-09-25) would otherwise end the run it was meant to be recorded by. Never loosen the
//     encoder guard to make a run pass.
//  4. BRAKE AT A NORMAL SEGMENT END ONLY, UNTIL REST; COAST EVERYWHERE ELSE. Every refusal and
//     every abort leaves every member at 0 V + Coast ON THE SAME TICK — a person may be reaching for
//     the robot (the calibrator's rule 6), and a braked robot cannot be pushed.
//  5. A MISSED TICK WHILE DRIVING IS A STOP: a clock gap past commandTimeout while Driving commands
//     0 V + Coast and refuses (MissedTick). Every powered volt is re-commanded every tick. A gap
//     while at rest or stopping powers nothing, and is not refused.
//  6. THE IMU'S YAW IS ACCUMULATED PER TICK through Angle::errorTo (the calibrator's rule 3), and
//     RECORDED ONLY — with the largest tick-to-tick step of every segment.
//  7. HOT MOTORS DO NOT START A SEGMENT: every member under tempGuard (45 °C, below HA-44's first
//     throttle step), else TooHot before anything is powered. The largest member temperature of
//     every segment is recorded.
//  8. THE FIT IS FED EVERY TICK, driving AND stopping, and ITS admission decides (feedforward_fit.hpp):
//     the stop's samples are excluded there as u = 0, which is what makes that exclusion
//     load-bearing on the plant (admitting them moves kS from 1.00 to 0.53).
//
// ── What it reports ─────────────────────────────────────────────────────────────────
// Per segment: how it ended; the samples admitted and excluded by reason, per side; each side's
// powered travel and STOPPING DISTANCE; the battery's minimum and mean; the largest member
// temperature; the encoders' curve ψ and the IMU's accumulated yaw at the powered end, and their
// difference; the largest tick-to-tick heading step; the segment's tick timing. Per side: the fit
// (FitResult). The combined gains (kA marked RECORDED, NOT APPLIED — the pipeline calls the cruise
// overload), the sides' kV asymmetry and its warning, and a warning for any segment that admitted
// no sample on a side (below, "What it does NOT refuse"). lastSample() exposes this tick's raw
// reading (time, volts, each side's median speed AND median position, battery, current, IMU
// heading), so a program can record the raw data (Part C's CSV) without this class allocating. A
// segment a refusal ended keeps how far it got — its seconds, last volts, travel, curve, IMU yaw and
// timing as last measured (its `end` stays None) — so the record after a refusal shows the numbers
// behind it.
//
// ── The clear space, and what it assumes (R5a's D9 and D11, corrected 2026-10-04) ───────
// clearAheadAndBehind() = segmentDistance + stopBound (48 in) and sidewaysExcursion(W) = the powered
// arc to the guard, segmentDistance·(1 − cos ψ_g)/ψ_g, PLUS stopBound (26.6 in at W = 12.5 in). The
// design's 14.6 in left the stop out: the curve guard is judged only while Driving, and a stop that
// keeps curving — on the host test plant a Done run at 46° a segment with stops near 11 in swept
// 19.7 in sideways — or curves harder than the powered part (one side braking short: a 13.1 in /
// 1.9 in stop after a 6° segment) is not bounded by the arc. What the two numbers assume:
//   * PER SEGMENT, from that segment's start pose. Each backward segment drives back along roughly
//     the path its forward segment took, so the robot stays near its start line; on the host test
//     plant the run's own excursion from its start pose stayed inside both bounds in every case
//     measured (development log, shulib-v2 branch, 2026-10-04). A robot that drives very
//     differently backward than forward could drift further over the run — nothing here tracks the
//     run's pose (an open question for the team lead, beside the one below).
//   * The powered part curves at a roughly CONSTANT rate (equal volts on fixed gains). The guard
//     bounds |ψ| on every tick, not the path's shape: a robot that turned ψ_g at once and then ran
//     straight would go segmentDistance·sin ψ_g (27.4 in) sideways — in practice the guard refuses
//     that early turn first (a kS of 2.2 V against 1.0 is refused CurveExceeded within the first
//     segment on the host test plant).
//   * Every stop stays inside stopBound. A stop past it is refused (StopTooLong) only once it has
//     happened: the robot has ALREADY rolled that far, so the lane's ends need room beyond the bound
//     for the one stop that exceeds it. The plant cannot say how far a V5 rolls or brakes (HA-53).
//
// ── What it does NOT refuse (open, for the team lead) ───────────────────────────────
// A segment that does not MOVE — something in front of the robot, a foot on it, a side whose
// motors cannot turn it — is not refused: it ends normally on RampTop (the ramp at
// rampMax, after up to rampMax/rampRate = 5 s of pushing) or on Timeout (a step after
// segmentTimeout, 8 s at stepVolts), with every sample in the dead band and so none admitted. The
// run then finishes on the segments that did move — a robot blocked in one direction finishes Done
// on the other direction's samples alone — and says so: any segment that admitted NO sample on a
// side sets CharacterizerReport::emptySegmentWarning and leads warning() with the segment, the side,
// its travel and how it ended (R5a's review of 2026-10-04 found such a run finishing with an empty
// warning; a side held at its current limit through a whole segment warns the same way). The
// drivetrain calibrator REFUSES the same situation (Refusal::SpinNoStart — "is it pinned, or on
// blocks?"); whether the GAINS step should too, and with what thresholds, is the team lead's
// decision before shulib Calibrate runs it (R5a Part C). The exposure until then is bounded:
// segmentTimeout per segment, every refusal and every normal end through the rules above.
//
// Tick-driven and non-blocking (10 ms on the robot); allocation-free; PROS-free and sim-free;
// raises no faults (what to do about a refusal is the program's policy); every runtime string is
// ASCII with no '%' ("pct") — PROS re-reads screen text as a format.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

#include "shulib/calibration/drive_characterization.hpp"
#include "shulib/calibration/drivetrain_calibrator.hpp"
#include "shulib/calibration/feedforward_fit.hpp"
#include "shulib/calibration/rest_capture.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/tick_timing_stats.hpp"
#include "shulib/hal/battery.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::calibration {

/// The four segments, in the order they run. APPEND-ONLY (the value is printed and recorded).
enum class SegmentKind : std::uint8_t {
    QuasiForward = 0,   ///< a ramp from 0 V at rampRate, forward
    QuasiBackward = 1,  ///< the same ramp, backward
    StepForward = 2,    ///< stepVolts held, forward
    StepBackward = 3,   ///< stepVolts held, backward — the hard reverse (HA-160)
};

/// How many segments a run has.
inline constexpr std::size_t kSegmentCount = 4;

/// A short, stable, ASCII name for a segment — the serial record's word and every message's.
[[nodiscard]] constexpr const char* segmentKindName(SegmentKind k) noexcept {
    switch (k) {
        case SegmentKind::QuasiForward: return "QuasiForward";
        case SegmentKind::QuasiBackward: return "QuasiBackward";
        case SegmentKind::StepForward: return "StepForward";
        case SegmentKind::StepBackward: return "StepBackward";
    }
    return "?";
}

/// How a segment's powered part ended. None = it did not end normally (a refusal, or still running).
/// APPEND-ONLY.
enum class SegmentEnd : std::uint8_t {
    None = 0,      ///< not ended normally
    Distance = 1,  ///< a side's travel reached segmentDistance
    RampTop = 2,   ///< the ramp reached rampMax
    Timeout = 3,   ///< segmentTimeout passed
};

/// A short, stable, ASCII word for how a segment ended (the serial record's word and the warning's).
[[nodiscard]] constexpr const char* segmentEndName(SegmentEnd e) noexcept {
    switch (e) {
        case SegmentEnd::None: return "none";
        case SegmentEnd::Distance: return "distance";
        case SegmentEnd::RampTop: return "ramp top";
        case SegmentEnd::Timeout: return "timeout";
    }
    return "?";
}

/// Where a run has got to. A run ends in exactly one of Done or Aborted.
enum class CharacterizerPhase : std::uint8_t {
    Idle,      ///< constructed, start() not yet called
    WaitImu,   ///< waiting for the IMU's boot calibration
    Rest,      ///< unpowered, waiting for the rest definition before the next segment
    Driving,   ///< POWERED: a segment's volts, re-commanded every tick
    Stopping,  ///< 0 V + Brake until rest; the stopping distance is being measured
    Done,      ///< result() is valid
    Aborted,   ///< refused or aborted; reason() says why; every member at 0 V + Coast
};

/// WHY a run ended in Aborted — a code a panel words and a test asserts. APPEND-ONLY.
enum class CharacterizerRefusal : std::uint8_t {
    None,           ///< not refused
    Aborted,        ///< abort() — the operator, or the program on the operator's behalf
    ImuNotReady,    ///< the IMU never became ready within imuReadyTimeout
    RestTimeout,    ///< nothing came to rest within restTimeout (before a segment, or after one)
    CurveExceeded,  ///< the encoders' curve passed ψ_g while driving (rule 3)
    Fight,          ///< the coupled-side monitor saw a member fight its side while driving
    MissedTick,     ///< tick() was not called within commandTimeout while driving
    TooHot,         ///< a member was at or over tempGuard before a segment
    StopTooLong,    ///< a side took more than stopBound to stop after a segment
    FitRefused,     ///< a side's fit did not come out Ok (the report names the side and verdict)
};

/// Every number the GAINS step uses. Its own are INVENTED for a first run and carried as fields so
/// an S2 serial log can move them with a logged line — PROVISIONAL, every one (A4: HA-170). The
/// rest definition, the IMU wait, the missed-tick ceiling and the fight monitor are the drivetrain
/// calibrator's, copied from CalibratorConfig's defaults so the two steps agree (A4: HA-136; the
/// monitor's HA-132/133).
struct CharacterizerConfig {
    /// The quasi-static ramp's rate, V/s. PROVISIONAL (A4: HA-170).
    double rampRate = 2.0;
    /// The ramp ends when it reaches this, V. PROVISIONAL (A4: HA-170).
    double rampMax = 10.0;
    /// The step segments' held volts. PROVISIONAL (A4: HA-170).
    double stepVolts = 6.0;
    /// A segment ends when EITHER side has travelled this far. 36 in — a CHOICE that sets the lane's
    /// length; whether it is long enough is S2's question. PROVISIONAL (A4: HA-170).
    units::Length segmentDistance{36.0};
    /// A segment ends after this long, whatever it has done. PROVISIONAL (A4: HA-170).
    units::Time segmentTimeout{8.0};
    /// The curve guard, as the side asymmetry it tolerates: ψ_g = segmentDistance × asymRefuse / W
    /// (rule 3). 0.30 — twice asymWarn, so a robot that warns still finishes. PROVISIONAL
    /// (A4: HA-170).
    double asymRefuse = 0.30;
    /// The sides' kV differing by more than this fraction of their mean is WARNED (a dragging side
    /// or a slipping member — a mechanical finding, not a tuning; R5a's decision D2). PROVISIONAL
    /// (A4: HA-170).
    double asymWarn = 0.15;
    /// A stopping distance longer than this refuses (StopTooLong) — and, with segmentDistance, sets
    /// the clear space the lane needs. PROVISIONAL (A4: HA-170).
    units::Length stopBound{12.0};
    /// A member at or over this temperature (°C) refuses a segment (TooHot). 45 °C — under HA-44's
    /// first throttle step at 55 °C. PROVISIONAL (A4: HA-170).
    double tempGuard = 45.0;
    /// |velocity| under this is a still motor (the calibrator's; A4: HA-136).
    units::AngularVelocity restFloor{CalibratorConfig{}.restFloor};
    /// |IMU yaw rate| under this is a still robot (the calibrator's; A4: HA-136).
    units::AngularVelocity restYawRateFloor{CalibratorConfig{}.restYawRateFloor};
    /// How long stillness must hold (the calibrator's; A4: HA-136).
    units::Time restSettleTime{CalibratorConfig{}.restSettleTime};
    /// How long to wait for stillness — before a segment, and for a stop — before refusing (the
    /// calibrator's; A4: HA-136).
    units::Time restTimeout{CalibratorConfig{}.restTimeout};
    /// How long to wait for the IMU's boot calibration (the calibrator's; A4: HA-136).
    units::Time imuReadyTimeout{CalibratorConfig{}.imuReadyTimeout};
    /// A clock gap past this while Driving is a missed tick (the calibrator's; A4: HA-136).
    units::Time commandTimeout{CalibratorConfig{}.commandTimeout};
    /// The coupled-side fight detector's thresholds — the drive program's, hal::MotorGroup's and the
    /// calibrator's (A4: HA-132/133).
    teleop::SideMonitorConfig monitor{};
    /// The fit's admission rules, verdicts and ranges (A4: HA-170).
    FitConfig fit{};
    /// The loop timing's nominal period and late budget; the nominal must equal fit.nominalDt.
    diag::TickTimingConfig timing{};

    /// ψ_g, radians: segmentDistance × asymRefuse / trackWidth (rule 3).
    [[nodiscard]] double curveGuard(units::Length trackWidth) const noexcept {
        return segmentDistance.value() * asymRefuse / trackWidth.value();
    }

    /// The clear space to EACH SIDE of a segment's start line, inches — the powered arc of a
    /// segment curving right up to the guard, segmentDistance·(1 − cos ψ_g)/ψ_g (R5a's D9), PLUS
    /// the whole stopBound: the stop is not curve-guarded (once the brake is on nothing can change
    /// its path) and may curve far more sharply than the powered part — one side braking short,
    /// the other rolling on — so its sideways share is bounded only by its length. 14.6 + 12 =
    /// 26.6 in at W = 12.5 in. The header's "The clear space" says what this bound assumes.
    [[nodiscard]] double sidewaysExcursion(units::Length trackWidth) const noexcept {
        const double g = curveGuard(trackWidth);
        const double arc = g > 0.0 ? segmentDistance.value() * (1.0 - std::cos(g)) / g : 0.0;
        return arc + stopBound.value();
    }

    /// The clear space ahead AND behind, inches: segmentDistance + stopBound (R5a's decision D11)
    /// — the most a segment's centre travels, powered and stopping (the header's "The clear space").
    [[nodiscard]] double clearAheadAndBehind() const noexcept {
        return segmentDistance.value() + stopBound.value();
    }

    /// True when the config can run on a drivetrain of effective width `trackWidth`: every field
    /// finite and positive where it must be; rampMax and stepVolts in (0, 12]; asymWarn in (0, 1)
    /// and asymRefuse ABOVE it (a curve guard that fired before the warning could be reached would
    /// refuse every robot that warns); ψ_g at most π (a bigger guard bounds nothing a segment can
    /// do); tempGuard in (0, 100); restTimeout > restSettleTime; the fit's config valid; the timing's
    /// nominal equal to fit.nominalDt and its budget above it. Else false naming the field in `why`
    /// (may be null).
    [[nodiscard]] bool valid(units::Length trackWidth, char* why, std::size_t cap) const {
        auto fail = [&](const char* text) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap, "%s", text);
            }
            return false;
        };
        auto pos = [](double v) { return std::isfinite(v) && v > 0.0; };
        const double rail = hal::kMaxMotorVoltage.value();
        if (!pos(trackWidth.value())) {
            return fail("CharacterizerConfig: the track width must be finite and > 0");
        }
        if (!pos(rampRate) || !pos(rampMax) || !(rampMax <= rail)) {
            return fail("CharacterizerConfig: rampRate > 0 and rampMax in (0, 12] V");
        }
        if (!pos(stepVolts) || !(stepVolts <= rail)) {
            return fail("CharacterizerConfig: stepVolts must be in (0, 12] V");
        }
        if (!pos(segmentDistance.value()) || !pos(segmentTimeout.value())) {
            return fail("CharacterizerConfig: segmentDistance and segmentTimeout must be > 0");
        }
        if (!pos(asymWarn) || !(asymWarn < 1.0)) {
            return fail("CharacterizerConfig: asymWarn must be in (0, 1)");
        }
        if (!std::isfinite(asymRefuse) || !(asymRefuse > asymWarn)) {
            return fail("CharacterizerConfig: asymRefuse must be above asymWarn, or the curve "
                        "guard refuses a robot before its asymmetry warning can be reached");
        }
        if (!(curveGuard(trackWidth) <= math::Angle::kPi)) {
            return fail("CharacterizerConfig: the curve guard segmentDistance x asymRefuse / W "
                        "must be at most pi");
        }
        if (!pos(stopBound.value())) {
            return fail("CharacterizerConfig: stopBound must be > 0");
        }
        if (!pos(tempGuard) || !(tempGuard < 100.0)) {
            return fail("CharacterizerConfig: tempGuard must be in (0, 100) C");
        }
        if (!pos(restFloor.value()) || !pos(restYawRateFloor.value())
            || !std::isfinite(restSettleTime.value()) || !(restSettleTime.value() >= 0.0)
            || !pos(imuReadyTimeout.value()) || !pos(commandTimeout.value())) {
            return fail("CharacterizerConfig: the rest floors, settle time, IMU wait and command "
                        "timeout must be finite and positive");
        }
        if (!std::isfinite(restTimeout.value()) || !(restTimeout.value() > restSettleTime.value())) {
            return fail("CharacterizerConfig: restTimeout must exceed restSettleTime");
        }
        if (!fit.valid(why, cap)) {
            return false;
        }
        if (!(timing.nominal.value() == fit.nominalDt)
            || !(timing.budget.value() > timing.nominal.value())) {
            return fail("CharacterizerConfig: timing.nominal must equal fit.nominalDt, and "
                        "timing.budget exceed it");
        }
        return true;
    }
};

/// Everything one segment did — one serial line each.
struct SegmentRecord {
    /// Which segment.
    SegmentKind kind = SegmentKind::QuasiForward;
    /// How its powered part ended (None if a refusal ended it).
    SegmentEnd end = SegmentEnd::None;
    /// True once its stop reached rest — stopIn is then final.
    bool stopped = false;
    /// Seconds from the segment's first powered tick to its end tick — or to the tick a refusal
    /// ended it on.
    double seconds = 0.0;
    /// The last volts commanded before the end: within one tick's ramp under rampMax for a ramp
    /// that reached its top (the end tick commands nothing), the step's level for a step.
    double lastVolts = 0.0;
    /// Samples the fit admitted, per side ([0] LEFT, [1] RIGHT) — driving and stopping.
    std::uint32_t admitted[2] = {0, 0};
    /// Samples excluded, per side, by reason (index = FitExclusion value − 1).
    std::array<std::uint32_t, kFitExclusionReasons> excluded[2]{};
    /// Each side's powered travel, inches (median, encoders; negative backward) — as last measured
    /// when a refusal ended the segment while Driving.
    double travelIn[2] = {0.0, 0.0};
    /// Each side's STOPPING distance, inches: from the end tick to rest (negative backward) — so far
    /// while Stopping, and the last measured when a refusal ended the stop.
    double stopIn[2] = {0.0, 0.0};
    /// The battery's lowest reading over the segment, volts — over the SAME readings as the mean:
    /// one per tick from the segment's first powered tick through its stop.
    double batteryMin = 0.0;
    /// The battery's mean over the segment, volts (the readings batteryMin is the least of).
    double batteryMean = 0.0;
    /// The hottest member reading over the segment, °C.
    double maxTemperature = 0.0;
    /// The encoders' curve ψ = (dR − dL)/W at the powered end, radians (positive = CCW).
    double encoderCurveRad = 0.0;
    /// The IMU's accumulated yaw over the same span, radians (recorded only — rule 3).
    double imuYawRad = 0.0;
    /// The largest |tick-to-tick heading step| over the segment, radians (HA-160 evidence).
    double maxHeadingStepRad = 0.0;
    /// The segment's tick timing (driving and stopping) — so far, when a refusal ended it.
    diag::TickTimingSummary timing{};

    /// imuYawRad − encoderCurveRad: how far the IMU and the encoders disagree about the curve
    /// (HA-160 evidence; never a refusal).
    [[nodiscard]] double imuMinusEncoderRad() const noexcept { return imuYawRad - encoderCurveRad; }
};

/// Every number behind a run, filled as far as the run got — worth reading after a refusal too.
struct CharacterizerReport {
    /// One record per segment started, in order.
    SegmentRecord segments[kSegmentCount]{};
    /// How many entries of `segments` are real.
    std::uint8_t segmentCount = 0;
    /// Each side's fit ([0] LEFT, [1] RIGHT) — filled when the last segment's stop reaches rest.
    FitResult fit[2]{};
    /// The combined gains: the field-by-field mean of the two fits (D2). Its kA is RECORDED, NOT
    /// APPLIED — the pipeline calls the cruise overload (header).
    control::FeedforwardGains combined{};
    /// |kV_L − kV_R| / their mean.
    double kVAsymmetry = 0.0;
    /// kVAsymmetry over CharacterizerConfig::asymWarn.
    bool asymmetryWarning = false;
    /// True when some segment admitted NO sample on a side (its `admitted[side]` is 0): that side's
    /// gains come from the other segments alone — a segment that could not move (something in the
    /// way, a pinned robot) or a side held at its current limit the whole segment. Warned, not
    /// refused (the header's "What it does NOT refuse"). Set on a finished run only.
    bool emptySegmentWarning = false;
    /// ψ_g, radians (rule 3), from the description's track width.
    double curveGuardRad = 0.0;
    /// The whole run's tick timing (every driving and stopping tick).
    diag::TickTimingSummary timing{};
};

/// This tick's raw reading, for a program to record (Part C's CSV) without the library allocating.
struct CharacterizerSample {
    /// False until the first driving tick; then true for every driving and stopping tick.
    bool valid = false;
    /// The clock, seconds.
    double t = 0.0;
    /// The segment (SegmentKind's value).
    std::uint8_t segment = 0;
    /// True while Driving, false while Stopping.
    bool powered = false;
    /// The volts commanded this tick (0 while stopping).
    double volts = 0.0;
    /// Each side's median surface speed, in/s.
    double speed[2] = {0.0, 0.0};
    /// Each side's median position, inches (cumulative — sidePosition()).
    double position[2] = {0.0, 0.0};
    /// The battery, volts.
    double battery = 0.0;
    /// Each side's largest |member current|, amperes.
    double maxCurrent[2] = {0.0, 0.0};
    /// The IMU's heading as read, radians (wrapped).
    double headingRad = 0.0;
};

/// What a panel paints this tick.
struct CharacterizerStatus {
    /// The phase.
    CharacterizerPhase phase = CharacterizerPhase::Idle;
    /// The segment under way or about to start (0-based; kSegmentCount once finished).
    std::uint8_t segment = 0;
    /// The volts in force (0 when not driving).
    double volts = 0.0;
    /// Each side's travel in the current segment, inches.
    double travelIn[2] = {0.0, 0.0};
    /// The encoders' curve so far in the current segment, radians.
    double curveRad = 0.0;
    /// Each side's stopping distance so far, inches (Stopping only).
    double stopIn[2] = {0.0, 0.0};
    /// True when the rest definition holds this tick.
    bool atRest = false;
};

/// The tick-driven GAINS step (header). Non-owning: the motors (through the sides' spans), the IMU,
/// the clock and the battery must outlive it. Not copyable. RUN AGAIN is a new object — nothing is
/// carried over by construction (the calibrator's rule).
class DriveCharacterizer {
public:
    /// `left`/`right`: each side's members and scale, from the drivetrain record (rule 1: the sign
    /// lives in SideMember). `trackWidth`: the record's EFFECTIVE width — ψ and ψ_g are computed with
    /// it (rule 3). Preconditions: both sides driveSideValid(), no port listed twice across them,
    /// config.valid(trackWidth).
    DriveCharacterizer(DriveSide left, DriveSide right, units::Length trackWidth, hal::IImu& imu,
                       hal::IClock& clock, hal::IBattery& battery,
                       const CharacterizerConfig& config = {})
        : sides_{left, right},
          width_{trackWidth.value()},
          imu_{imu},
          clock_{clock},
          battery_{battery},
          cfg_{config},
          fit_{FeedforwardFit{config.fit}, FeedforwardFit{config.fit}},
          runTiming_{config.timing},
          segTiming_{config.timing},
          monitor_{teleop::CoupledSideMonitor{config.monitor},
                   teleop::CoupledSideMonitor{config.monitor}} {
        SHULIB_PRECONDITION(driveSideValid(left) && driveSideValid(right),
                            "DriveCharacterizer: each side needs 1..21 non-null members, sign +-1, "
                            "port 1..21 and a positive scale");
        SHULIB_PRECONDITION(cfg_.valid(trackWidth, nullptr, 0),
                            "DriveCharacterizer: the CharacterizerConfig is invalid for this track "
                            "width");
        SHULIB_PRECONDITION(portsUnique(), "DriveCharacterizer: a port is listed twice");
        guard_ = cfg_.curveGuard(trackWidth);
        report_.curveGuardRad = guard_;
    }

    /// Not copyable: it holds references to hardware it does not own, and a copy would be a second
    /// state machine commanding the same motors.
    DriveCharacterizer(const DriveCharacterizer&) = delete;
    /// Not copy-assignable, for the same reason.
    DriveCharacterizer& operator=(const DriveCharacterizer&) = delete;

    /// Begin a run: every member to 0 V + Coast, then WaitImu. `provenance` (the build stamp or a
    /// session note) is copied into the result's fixed buffer, cut to fit; no pointer is kept.
    /// Precondition: the run is Idle.
    void start(const char* provenance = "") {
        SHULIB_PRECONDITION(phase_ == CharacterizerPhase::Idle,
                            "DriveCharacterizer::start: a run is already under way");
        std::snprintf(provenance_, sizeof provenance_, "%s",
                      provenance != nullptr ? provenance : "");
        safeState();
        enter(CharacterizerPhase::WaitImu);
    }

    /// One control tick. Cheap and non-blocking; the caller paces it (10 ms). A no-op before start()
    /// and after the run ends.
    void tick() {
        if (phase_ == CharacterizerPhase::Idle || phase_ == CharacterizerPhase::Done
            || phase_ == CharacterizerPhase::Aborted) {
            return;
        }
        const units::Time now = clock_.now();
        if (ticked_ && phase_ == CharacterizerPhase::Driving
            && (now.value() - lastTick_) > cfg_.commandTimeout.value()) {
            // Rule 5: a powered segment holds its volts between ticks, so a caller that stops
            // ticking leaves the motors running. Stop them and say so.
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "a tick was missed while driving: %.3f s since the last one, ceiling "
                          "%.3f s -- motors stopped",
                          now.value() - lastTick_, cfg_.commandTimeout.value());
            refuse(CharacterizerRefusal::MissedTick, 0, msg);
            return;
        }
        lastTick_ = now.value();
        ticked_ = true;
        switch (phase_) {
            case CharacterizerPhase::WaitImu: tickWaitImu(now); break;
            case CharacterizerPhase::Rest: tickRest(now); break;
            case CharacterizerPhase::Driving: tickDriving(now); break;
            case CharacterizerPhase::Stopping: tickStopping(now); break;
            default: break;
        }
    }

    /// Stop now: every member to 0 V + Coast (never Brake — rule 4), Aborted with
    /// CharacterizerRefusal::Aborted and `why` as the reason. Allowed in any phase, including Done
    /// (it then discards the result).
    void abort(const char* why = "aborted by the operator") {
        refuse(CharacterizerRefusal::Aborted, 0, why != nullptr ? why : "aborted");
    }

    /// Where the run has got to.
    [[nodiscard]] CharacterizerPhase phase() const noexcept { return phase_; }
    /// Which refusal ended the run (None while it runs or once it is Done).
    [[nodiscard]] CharacterizerRefusal refusal() const noexcept { return refusal_; }
    /// The port the refusal names (a hot motor, a fighting member, one that would not rest), or 0.
    [[nodiscard]] int refusalPort() const noexcept { return refusalPort_; }
    /// Why the run was refused — empty while it runs or once it is Done. ASCII, no '%'.
    [[nodiscard]] const char* reason() const noexcept { return reason_; }
    /// The warnings a FINISHED run still owes the operator — a segment that admitted no sample on a
    /// side (CharacterizerReport::emptySegmentWarning) first, then the sides' kV asymmetry past
    /// asymWarn, joined by "; ". Empty when there is none. ASCII, no '%'.
    [[nodiscard]] const char* warning() const noexcept { return warning_; }
    /// Every number behind the run, filled as far as it got.
    [[nodiscard]] const CharacterizerReport& report() const noexcept { return report_; }
    /// What a panel paints this tick.
    [[nodiscard]] const CharacterizerStatus& status() const noexcept { return status_; }
    /// This tick's raw reading (CharacterizerSample).
    [[nodiscard]] const CharacterizerSample& lastSample() const noexcept { return sample_; }
    /// The config in force.
    [[nodiscard]] const CharacterizerConfig& config() const noexcept { return cfg_; }

    /// The measured characterization: source Measured, the GAINS section filled (each side's fit and
    /// member count, the run's timing), no REST section. Precondition: phase() == Done.
    [[nodiscard]] const DriveCharacterization& result() const {
        SHULIB_PRECONDITION(phase_ == CharacterizerPhase::Done,
                            "DriveCharacterizer::result: the run did not finish -- read reason()");
        return result_;
    }

private:
    // ── phase bodies ───────────────────────────────────────────────────────────────

    void tickWaitImu(units::Time now) {
        if (imu_.isReady()) {
            enter(CharacterizerPhase::Rest);
            return;
        }
        if (now.value() - phaseStart_ > cfg_.imuReadyTimeout.value()) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg, "the IMU never became ready within %.1f s",
                          cfg_.imuReadyTimeout.value());
            refuse(CharacterizerRefusal::ImuNotReady, 0, msg);
        }
    }

    void tickRest(units::Time now) {
        const int moving = firstMovingPort();
        status_.atRest = moving == 0 && imuStill();
        if (status_.atRest) {
            if (!resting_) {
                resting_ = true;
                restSince_ = now.value();
            }
            if (now.value() - restSince_ >= cfg_.restSettleTime.value()) {
                if (!coolEnough()) {
                    return;  // refused TooHot, nothing powered
                }
                beginSegment(now);
            }
            return;
        }
        resting_ = false;
        if (now.value() - phaseStart_ > cfg_.restTimeout.value()) {
            refuseNoRest(moving, "before a segment");
        }
    }

    void tickDriving(units::Time now) {
        SegmentRecord& seg = report_.segments[segIndex_];
        const Reading r = read();
        feedFits(now, r, seg);
        accumulate(now, r, seg);
        const double travel[2] = {sideTravel(sides_[0], base_[0]), sideTravel(sides_[1], base_[1])};
        curve_ = (travel[1] - travel[0]) / width_;  // rule 3: the ENCODERS, the description's W
        status_.travelIn[0] = travel[0];
        status_.travelIn[1] = travel[1];
        status_.curveRad = curve_;
        if (fought()) {
            return;
        }
        if (std::abs(curve_) > guard_) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the robot curved %.1f deg by its encoders during %s (guard %.1f deg): "
                          "LEFT travelled %+.1f in, RIGHT %+.1f in -- a dragging side, a slipping "
                          "member or something in the way",
                          curve_ / kDegreesToRadians, segmentKindName(seg.kind),
                          guard_ / kDegreesToRadians, travel[0] + 0.0, travel[1] + 0.0);
            refuse(CharacterizerRefusal::CurveExceeded, 0, msg);
            return;
        }
        const double elapsed = now.value() - segStart_;
        SegmentEnd end = SegmentEnd::None;
        double volts = voltsAt(seg.kind, elapsed);
        const double limit = cfg_.segmentDistance.value();
        if (std::abs(travel[0]) >= limit || std::abs(travel[1]) >= limit) {
            end = SegmentEnd::Distance;  // PER SIDE, never the mean: the faster side runs out first
        } else if (elapsed >= cfg_.segmentTimeout.value()) {
            end = SegmentEnd::Timeout;
        } else if (isQuasi(seg.kind) && std::abs(volts) >= cfg_.rampMax) {
            end = SegmentEnd::RampTop;
        }
        if (end != SegmentEnd::None) {
            seg.end = end;
            seg.seconds = elapsed;
            seg.lastVolts = volts_;
            seg.travelIn[0] = travel[0];
            seg.travelIn[1] = travel[1];
            seg.encoderCurveRad = curve_;
            seg.imuYawRad = yaw_;
            beginStop(now);
            volts = 0.0;
        } else {
            command(volts);
        }
        remember(now, r, volts, end == SegmentEnd::None);
    }

    void tickStopping(units::Time now) {
        SegmentRecord& seg = report_.segments[segIndex_];
        const Reading r = read();
        feedFits(now, r, seg);  // u = 0: the fit excludes them as Stop (rule 8)
        accumulate(now, r, seg);
        brakeAll();             // re-commanded every tick, until rest
        const double stop[2] = {sideTravel(sides_[0], stopBase_[0]),
                                sideTravel(sides_[1], stopBase_[1])};
        status_.stopIn[0] = stop[0];
        status_.stopIn[1] = stop[1];
        seg.stopIn[0] = stop[0];  // so far — final once `stopped`; a refusal keeps the latest
        seg.stopIn[1] = stop[1];
        remember(now, r, 0.0, false);
        const int moving = firstMovingPort();
        status_.atRest = moving == 0 && imuStill();
        if (status_.atRest) {
            if (!resting_) {
                resting_ = true;
                restSince_ = now.value();
            }
            if (now.value() - restSince_ >= cfg_.restSettleTime.value()) {
                finishStop(stop);
            }
            return;
        }
        resting_ = false;
        if (now.value() - phaseStart_ > cfg_.restTimeout.value()) {
            if (std::max(std::abs(stop[0]), std::abs(stop[1])) > cfg_.stopBound.value()) {
                refuseStopTooLong(stop, true);
            } else {
                refuseNoRest(moving, "after a segment");
            }
        }
    }

    // ── transitions ────────────────────────────────────────────────────────────────

    void beginSegment(units::Time now) {
        SegmentRecord& seg = report_.segments[segIndex_];
        seg = SegmentRecord{};
        seg.kind = static_cast<SegmentKind>(segIndex_);
        report_.segmentCount = static_cast<std::uint8_t>(segIndex_ + 1);
        for (std::size_t s = 0; s < 2; ++s) {
            for (std::size_t k = 0; k < sides_[s].members.size(); ++k) {
                base_[s][k] = sides_[s].members[k].motor->position().value();
            }
            monitor_[s].reset();
        }
        // This tick's ONE reading: the IMU's baseline, the battery's first value, the sample.
        const Reading r0 = read();
        yaw_ = 0.0;
        curve_ = 0.0;
        prevHeading_ = r0.heading;
        segStart_ = now.value();
        segTiming_ = diag::TickTimingStats{cfg_.timing};
        (void)segTiming_.tick(now);
        runTiming_.rebaseline();  // the rest before a segment is a deliberate gap, not a tick
        (void)runTiming_.tick(now);
        // The battery's min and mean are over the SAME readings: this tick's, then accumulate()'s.
        batterySum_ = r0.battery;
        batteryTicks_ = 1;
        seg.batteryMin = r0.battery;
        seg.batteryMean = r0.battery;
        seg.maxTemperature = r0.maxTemperature;
        status_.travelIn[0] = 0.0;  // the panel's numbers are THIS segment's from its first tick
        status_.travelIn[1] = 0.0;
        status_.curveRad = 0.0;
        status_.stopIn[0] = 0.0;
        status_.stopIn[1] = 0.0;
        enter(CharacterizerPhase::Driving);
        const double volts = voltsAt(seg.kind, 0.0);
        command(volts);
        remember(now, r0, volts, true);
    }

    void beginStop(units::Time now) {
        (void)now;
        for (std::size_t s = 0; s < 2; ++s) {
            for (std::size_t k = 0; k < sides_[s].members.size(); ++k) {
                stopBase_[s][k] = sides_[s].members[k].motor->position().value();
            }
        }
        brakeAll();  // rule 4: 0 V + Brake at a NORMAL end, until rest
        volts_ = 0.0;
        status_.volts = 0.0;
        enter(CharacterizerPhase::Stopping);
    }

    void finishStop(const double* stop) {
        SegmentRecord& seg = report_.segments[segIndex_];
        seg.stopIn[0] = stop[0];
        seg.stopIn[1] = stop[1];
        seg.stopped = true;
        seg.timing = segTiming_.summary();
        safeState();  // at rest: Brake off, Coast (rule 4)
        prevValid_ = false;
        if (std::max(std::abs(stop[0]), std::abs(stop[1])) > cfg_.stopBound.value()) {
            refuseStopTooLong(stop, false);
            return;
        }
        ++segIndex_;
        if (segIndex_ < kSegmentCount) {
            enter(CharacterizerPhase::Rest);
            return;
        }
        finishRun();
    }

    void finishRun() {
        report_.timing = runTiming_.summary();
        report_.fit[0] = fit_[0].result();
        report_.fit[1] = fit_[1].result();
        const bool ok[2] = {report_.fit[0].verdict == FitVerdict::Ok,
                            report_.fit[1].verdict == FitVerdict::Ok};
        DriveCharacterization c;
        c.source = CharacterizationSource::Measured;
        c.hasGains = true;
        SideCharacterization* out[2] = {&c.left, &c.right};
        for (std::size_t s = 0; s < 2; ++s) {
            const FitResult& f = report_.fit[s];
            out[s]->gains = f.gains;
            out[s]->residualRms = f.residualRms;
            out[s]->alpha = f.alpha;
            out[s]->voltsMin = f.voltsMin;
            out[s]->voltsMax = f.voltsMax;
            out[s]->samples = f.samples;
            out[s]->members = static_cast<std::uint8_t>(sides_[s].members.size());
            out[s]->kABelowResolution = f.kABelowResolution;
        }
        c.rampTiming = report_.timing;
        std::snprintf(c.provenance, sizeof c.provenance, "%s", provenance_);
        report_.combined = c.combined();
        report_.kVAsymmetry = c.kVAsymmetry();
        report_.asymmetryWarning = report_.kVAsymmetry > cfg_.asymWarn;
        if (!ok[0] || !ok[1]) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the fit is refused -- LEFT %s (residual %.2f in/s over %u samples), "
                          "RIGHT %s (residual %.2f in/s over %u samples); nothing was measured",
                          fitVerdictName(report_.fit[0].verdict), report_.fit[0].residualRms,
                          static_cast<unsigned>(report_.fit[0].samples),
                          fitVerdictName(report_.fit[1].verdict), report_.fit[1].residualRms,
                          static_cast<unsigned>(report_.fit[1].samples));
            refuse(CharacterizerRefusal::FitRefused, 0, msg);
            return;
        }
        // A segment that admitted nothing on a side: the fit is whole without it, so the run
        // finishes — but say so, naming the first such segment (the header's "What it does NOT
        // refuse"; R5a's review of 2026-10-04: a robot blocked in one direction used to finish with
        // an empty warning).
        std::size_t empty = 0;
        std::size_t firstSeg = 0;
        std::size_t firstSide = 0;
        for (std::size_t k = 0; k < report_.segmentCount; ++k) {
            for (std::size_t s = 0; s < 2; ++s) {
                if (report_.segments[k].admitted[s] == 0U) {
                    if (empty == 0) {
                        firstSeg = k;
                        firstSide = s;
                    }
                    ++empty;
                }
            }
        }
        report_.emptySegmentWarning = empty > 0;
        std::size_t used = 0;
        if (report_.emptySegmentWarning) {
            const SegmentRecord& seg = report_.segments[firstSeg];
            char more[48] = "";
            if (empty > 1) {
                std::snprintf(more, sizeof more, ", and %u more segment-sides likewise",
                              static_cast<unsigned>(empty - 1));
            }
            const int n = std::snprintf(
                warning_, sizeof warning_,
                "%s admitted no %s sample (travel %+.1f in, ended on %s)%s -- that side's gains "
                "come from its other segments alone: something in the way, a pinned robot, or a "
                "side held at its current limit?",
                segmentKindName(seg.kind), firstSide == 0 ? "LEFT" : "RIGHT",
                seg.travelIn[firstSide] + 0.0, segmentEndName(seg.end), more);
            used = n > 0 ? std::min(static_cast<std::size_t>(n), sizeof warning_ - 1) : 0;
        }
        if (report_.asymmetryWarning) {
            std::snprintf(warning_ + used, sizeof warning_ - used,
                          "%sthe sides' kV differ by %.1f pct (LEFT %.4f, RIGHT %.4f; warning past "
                          "%.1f pct) -- a dragging side or a slipping member is a mechanical "
                          "finding, not a tuning",
                          used > 0 ? "; " : "", 100.0 * report_.kVAsymmetry, c.left.gains.kV,
                          c.right.gains.kV, 100.0 * cfg_.asymWarn);
        }
        result_ = c;
        phase_ = CharacterizerPhase::Done;
        status_.phase = phase_;
        status_.segment = static_cast<std::uint8_t>(kSegmentCount);
    }

    // ── helpers ────────────────────────────────────────────────────────────────────

    /// One tick's readings of both sides, the battery and the IMU.
    struct Reading {
        double speed[2] = {0.0, 0.0};
        double maxCurrent[2] = {0.0, 0.0};
        double maxTemperature = 0.0;
        double battery = 0.0;
        math::Angle heading{};
    };

    [[nodiscard]] Reading read() const {
        Reading r;
        for (std::size_t s = 0; s < 2; ++s) {
            r.speed[s] = sideSpeed(sides_[s]);
            for (const SideMember& m : sides_[s].members) {
                r.maxCurrent[s] = std::max(r.maxCurrent[s], std::abs(m.motor->current().value()));
                r.maxTemperature = std::max(r.maxTemperature, m.motor->temperature());
            }
        }
        r.battery = battery_.voltage().value();
        r.heading = imu_.heading();
        return r;
    }

    /// Hand each side's sample (last tick's speed and volts, this tick's speed) to its fit, and
    /// count the verdict in the segment's record (rule 8: the fit's admission decides).
    void feedFits(units::Time now, const Reading& r, SegmentRecord& seg) {
        if (!prevValid_) {
            return;
        }
        for (std::size_t s = 0; s < 2; ++s) {
            const FitSample sample{prevSpeed_[s], prevVolts_, r.speed[s], now.value() - prevT_,
                                   r.maxCurrent[s]};
            const FitExclusion why = fit_[s].add(sample);
            if (why == FitExclusion::None) {
                ++seg.admitted[s];
            } else {
                ++seg.excluded[s][static_cast<std::size_t>(why) - 1];
            }
        }
    }

    /// The per-segment and per-run accumulators: timing, battery, temperature, the IMU's yaw.
    void accumulate(units::Time now, const Reading& r, SegmentRecord& seg) {
        (void)segTiming_.tick(now);
        (void)runTiming_.tick(now);
        batterySum_ += r.battery;
        ++batteryTicks_;
        seg.batteryMin = std::min(seg.batteryMin, r.battery);
        seg.batteryMean = batterySum_ / static_cast<double>(batteryTicks_);
        seg.maxTemperature = std::max(seg.maxTemperature, r.maxTemperature);
        const double step = prevHeading_.errorTo(r.heading);  // rule 6: per tick, through errorTo
        prevHeading_ = r.heading;
        yaw_ += step;
        seg.maxHeadingStepRad = std::max(seg.maxHeadingStepRad, std::abs(step));
    }

    /// Keep this tick's reading and command for the next tick's fit sample, and expose it.
    void remember(units::Time now, const Reading& r, double volts, bool powered) {
        prevSpeed_[0] = r.speed[0];
        prevSpeed_[1] = r.speed[1];
        prevVolts_ = volts;
        prevT_ = now.value();
        prevValid_ = true;
        sample_.valid = true;
        sample_.t = now.value();
        sample_.segment = static_cast<std::uint8_t>(segIndex_);
        sample_.powered = powered;
        sample_.volts = volts;
        sample_.battery = r.battery;
        sample_.headingRad = r.heading.radians();
        for (std::size_t s = 0; s < 2; ++s) {
            sample_.speed[s] = r.speed[s];
            sample_.position[s] = sidePosition(sides_[s]);
            sample_.maxCurrent[s] = r.maxCurrent[s];
        }
    }

    /// Run the shared coupled-side monitor over both sides; a PERSISTED disagreement refuses,
    /// naming the member's port. Returns true when it refused.
    [[nodiscard]] bool fought() {
        for (std::size_t s = 0; s < 2; ++s) {
            const std::span<const SideMember> members = sides_[s].members;
            teleop::MemberSample samples[hal::kMaxSmartPorts];
            for (std::size_t k = 0; k < members.size(); ++k) {
                // The sign is applied HERE, once (rule 1).
                samples[k] = teleop::MemberSample{
                    static_cast<double>(members[k].sign) * members[k].motor->velocity().value(),
                    true};
            }
            const teleop::SideVerdict v = monitor_[s].update(
                prevVolts_, std::span<const teleop::MemberSample>{samples, members.size()});
            if (v.persistedCount == 0) {
                continue;
            }
            for (std::size_t k = 0; k < members.size(); ++k) {
                if ((v.persistedMask & (std::uint32_t{1} << k)) != 0U) {
                    char msg[kReasonCap];
                    std::snprintf(msg, sizeof msg,
                                  "port %d fought its %s side-mates for the whole persistence "
                                  "window during %s -- stop and check that motor",
                                  static_cast<int>(members[k].port), s == 0 ? "LEFT" : "RIGHT",
                                  segmentKindName(report_.segments[segIndex_].kind));
                    refuse(CharacterizerRefusal::Fight, members[k].port, msg);
                    return true;
                }
            }
        }
        return false;
    }

    /// Rule 7: every member under tempGuard, else TooHot naming the hottest, nothing powered.
    [[nodiscard]] bool coolEnough() {
        const SideMember* hottest = nullptr;
        double hot = 0.0;
        for (const DriveSide& side : sides_) {
            for (const SideMember& m : side.members) {
                const double t = m.motor->temperature();
                if (t >= cfg_.tempGuard && (hottest == nullptr || t > hot)) {
                    hottest = &m;
                    hot = t;
                }
            }
        }
        if (hottest == nullptr) {
            return true;
        }
        char msg[kReasonCap];
        std::snprintf(msg, sizeof msg,
                      "port %d is at %.1f C before %s (guard %.1f C) -- let the motors cool, then "
                      "run GAINS again",
                      static_cast<int>(hottest->port), hot,
                      segmentKindName(static_cast<SegmentKind>(segIndex_)), cfg_.tempGuard);
        refuse(CharacterizerRefusal::TooHot, hottest->port, msg);
        return false;
    }

    void refuseNoRest(int moving, const char* where) {
        char msg[kReasonCap];
        if (moving != 0) {
            std::snprintf(msg, sizeof msg,
                          "port %d is still turning after %.1f s of waiting for rest %s -- a motor "
                          "that reports motion while nothing is driven is a frozen reading or a "
                          "moving robot",
                          moving, cfg_.restTimeout.value(), where);
        } else {
            std::snprintf(msg, sizeof msg,
                          "the robot is still rotating at %.1f deg/s after %.1f s of waiting for "
                          "rest %s -- hands off the robot, and keep it on the floor",
                          imu_.yawRate().value() / kDegreesToRadians, cfg_.restTimeout.value(),
                          where);
        }
        refuse(CharacterizerRefusal::RestTimeout, moving, msg);
    }

    void refuseStopTooLong(const double* stop, bool stillRolling) {
        char msg[kReasonCap];
        std::snprintf(msg, sizeof msg,
                      "after %s the robot took LEFT %+.1f in / RIGHT %+.1f in to stop under Brake "
                      "(bound %.1f in)%s -- give it more room, or check the floor and the brake",
                      segmentKindName(report_.segments[segIndex_].kind), stop[0] + 0.0, stop[1] + 0.0,
                      cfg_.stopBound.value(), stillRolling ? " and was still rolling" : "");
        refuse(CharacterizerRefusal::StopTooLong, 0, msg);
    }

    /// The volts segment `kind` commands `elapsed` seconds after it began.
    [[nodiscard]] double voltsAt(SegmentKind kind, double elapsed) const noexcept {
        switch (kind) {
            case SegmentKind::QuasiForward: return cfg_.rampRate * elapsed;
            case SegmentKind::QuasiBackward: return -cfg_.rampRate * elapsed;
            case SegmentKind::StepForward: return cfg_.stepVolts;
            case SegmentKind::StepBackward: return -cfg_.stepVolts;
        }
        return 0.0;
    }

    [[nodiscard]] static bool isQuasi(SegmentKind k) noexcept {
        return k == SegmentKind::QuasiForward || k == SegmentKind::QuasiBackward;
    }

    /// Both sides, every member, `volts` × its sign; Coast (the mode only a zero would use).
    void command(double volts) {
        for (const DriveSide& side : sides_) {
            for (const SideMember& m : side.members) {
                m.motor->setBrakeMode(hal::BrakeMode::Coast);
                m.motor->setVoltage(units::Voltage{static_cast<double>(m.sign) * volts});
            }
        }
        volts_ = volts;
        status_.volts = volts;
    }

    /// Rule 4, normal end: every member 0 V + BRAKE.
    void brakeAll() {
        for (const DriveSide& side : sides_) {
            for (const SideMember& m : side.members) {
                m.motor->setVoltage(units::Voltage{0.0});
                m.motor->setBrakeMode(hal::BrakeMode::Brake);
            }
        }
    }

    /// Rule 4, everywhere else: every member 0 V + COAST.
    void safeState() {
        for (const DriveSide& side : sides_) {
            for (const SideMember& m : side.members) {
                m.motor->setVoltage(units::Voltage{0.0});
                m.motor->setBrakeMode(hal::BrakeMode::Coast);
            }
        }
        volts_ = 0.0;
        status_.volts = 0.0;
    }

    /// The robot's half of the rest definition: the IMU's yaw rate under restYawRateFloor.
    [[nodiscard]] bool imuStill() const {
        return std::abs(imu_.yawRate().value()) <= cfg_.restYawRateFloor.value();
    }

    /// The motors' half: the first member turning faster than restFloor (its port), or 0.
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

    void enter(CharacterizerPhase p) {
        phase_ = p;
        phaseStart_ = clock_.now().value();
        resting_ = false;
        status_.phase = p;
        status_.segment = static_cast<std::uint8_t>(segIndex_);
    }

    /// Every refusal comes through here — the reason, the code, the port, 0 V + COAST on every
    /// member (rule 4), Aborted — which makes "Coast on every refusal, on the same tick" a property
    /// of one function rather than of each site.
    void refuse(CharacterizerRefusal code, int port, const char* text) {
        std::snprintf(reason_, sizeof reason_, "%s", text);
        refusal_ = code;
        refusalPort_ = port;
        keepSegmentSoFar();  // before safeState() zeroes volts_
        safeState();
        phase_ = CharacterizerPhase::Aborted;
        status_.phase = phase_;
    }

    /// A refusal that ends a segment part-way keeps how far it got (header): while Driving, its
    /// seconds, last volts, travel, curve and IMU yaw as last measured (`end` stays None); while
    /// Stopping, the stop distance is already kept tick by tick. Either way, the timing so far.
    /// Nothing for a refusal between segments — the last one is complete.
    void keepSegmentSoFar() {
        if (phase_ != CharacterizerPhase::Driving && phase_ != CharacterizerPhase::Stopping) {
            return;
        }
        SegmentRecord& seg = report_.segments[segIndex_];
        if (phase_ == CharacterizerPhase::Driving) {
            seg.seconds = clock_.now().value() - segStart_;
            seg.lastVolts = volts_;
            seg.travelIn[0] = status_.travelIn[0];
            seg.travelIn[1] = status_.travelIn[1];
            seg.encoderCurveRad = curve_;
            seg.imuYawRad = yaw_;
        }
        if (!seg.stopped) {
            seg.timing = segTiming_.summary();
        }
    }

    // ── state ──────────────────────────────────────────────────────────────────────
    static constexpr std::size_t kReasonCap = 256;

    DriveSide sides_[2];
    double width_;
    hal::IImu& imu_;
    hal::IClock& clock_;
    hal::IBattery& battery_;
    CharacterizerConfig cfg_;
    FeedforwardFit fit_[2];
    diag::TickTimingStats runTiming_;
    diag::TickTimingStats segTiming_;
    teleop::CoupledSideMonitor monitor_[2];
    double guard_ = 0.0;

    CharacterizerPhase phase_ = CharacterizerPhase::Idle;
    CharacterizerRefusal refusal_ = CharacterizerRefusal::None;
    int refusalPort_ = 0;
    char reason_[kReasonCap] = "";
    char warning_[2 * kReasonCap] = "";  // two warnings may be owed at once (warning())
    char provenance_[96] = "";
    CharacterizerReport report_{};
    CharacterizerStatus status_{};
    CharacterizerSample sample_{};
    DriveCharacterization result_{};

    std::size_t segIndex_ = 0;
    double phaseStart_ = 0.0;
    double segStart_ = 0.0;
    double lastTick_ = 0.0;
    bool ticked_ = false;
    bool resting_ = false;
    double restSince_ = 0.0;
    double volts_ = 0.0;

    double base_[2][hal::kMaxSmartPorts]{};
    double stopBase_[2][hal::kMaxSmartPorts]{};
    math::Angle prevHeading_{};
    double yaw_ = 0.0;
    double curve_ = 0.0;
    double batterySum_ = 0.0;
    std::uint32_t batteryTicks_ = 0;

    bool prevValid_ = false;
    double prevSpeed_[2] = {0.0, 0.0};
    double prevVolts_ = 0.0;
    double prevT_ = 0.0;
};

/// A short, stable, ASCII name for a refusal — the serial record's code.
[[nodiscard]] constexpr const char* characterizerRefusalName(CharacterizerRefusal r) noexcept {
    switch (r) {
        case CharacterizerRefusal::None: return "NONE";
        case CharacterizerRefusal::Aborted: return "ABORTED";
        case CharacterizerRefusal::ImuNotReady: return "IMU_NOT_READY";
        case CharacterizerRefusal::RestTimeout: return "REST_TIMEOUT";
        case CharacterizerRefusal::CurveExceeded: return "CURVE_EXCEEDED";
        case CharacterizerRefusal::Fight: return "FIGHT";
        case CharacterizerRefusal::MissedTick: return "MISSED_TICK";
        case CharacterizerRefusal::TooHot: return "TOO_HOT";
        case CharacterizerRefusal::StopTooLong: return "STOP_TOO_LONG";
        case CharacterizerRefusal::FitRefused: return "FIT_REFUSED";
    }
    return "?";
}

}  // namespace shulib::calibration
