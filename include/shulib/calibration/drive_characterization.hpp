#pragma once
//
// DriveCharacterization — what the GAINS and REST steps of shulib Calibrate MEASURE, as data (chunk
// R5a Part A, 2026-09-29). drive_characterizer.hpp (the GAINS step: each side's kS/kV/kA) and
// rest_capture.hpp (the REST step: the IMU's drift and noise, the encoders' noise at rest, the loop's
// timing) produce it; Part B of R5a persists it on the SD card beside the drivetrain record, bound
// to that record, and teaches the robot builder to consume the gains. NOTHING CONSUMES IT YET: until
// Part B, no program reads it and every drive runs on the placeholder gains (A4 register HA-45).
//
// NOTHING IN IT HAS BEEN MEASURED ON A ROBOT. The host test plant fills it with the plant's own
// constants; a robot session (S2 gains, S3 noise) is what will put real numbers here.
//
// ── Why TWO sides, and what "combined" means (R5a's decision D2) ───────────────────────
// The motion layer applies ONE control::Feedforward to every wheel (feedforward.hpp; the command
// pipeline's step 7). The two sides are fitted and kept SEPARATELY anyway — a dragging side or a
// slipping member is a mechanical finding, and averaging it away before anyone saw it would hide
// it — and combined() is their field-by-field MEAN, which is what Part B hands the pipeline.
// kVAsymmetry() says how far apart the sides are; the characterizer warns past 15 %.
// OF THE MEAN, ONLY kS AND kV REACH A MOTOR: the pipeline calls the cruise overload
// Feedforward::calculate(v) and WheelSpeeds has no acceleration channel, so kA is RECORDED (for the
// S2 log and R6's plant back-fit, where τ = kA/kV) and NOT APPLIED (D2, D10).
//
// ── Laid out to be SERIALIZED (Part B's record) ─────────────────────────────────────
// Fixed-width fields, bounded arrays, no pointers, no std::string, no allocation, constexpr-
// constructible — config/robot_config.hpp's rule for everything a RobotConfig carries, because Part
// B embeds this struct in RobotConfig BY VALUE (that header's ruling B1: one representation, never a
// mirror). The section flags say which halves hold data, so a REST run can be merged into a record
// that already holds a GAINS run without erasing it (Part B's mergeSections). `source` and every
// enum here are APPEND-ONLY: their values are persisted.
//
// ── The one thing that must never be done with `rest` (A4 register HA-173) ───────────
// The drift is a PER-BOOT bias of the IMU (HA-20): one boot's number is wrong at the next boot. It is
// a sample for the register, reported and kept — NEVER applied as a heading correction.
//
// PROS-free, sim-free, allocation-free. valid() writes ASCII with no '%' (a panel may paint it).

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "shulib/control/feedforward.hpp"
#include "shulib/diag/tick_timing_stats.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/motor.hpp"

namespace shulib::calibration {

/// Where a characterization came from. APPEND-ONLY and persisted: a new source goes at the end.
enum class CharacterizationSource : std::uint8_t {
    Unset = 0,     ///< nothing measured — the placeholders (HA-45) are in force
    Measured = 1,  ///< measured by the robot, in shulib Calibrate (GAINS and/or REST)
};

/// One drive side's fitted gains and the facts behind them (calibration::FitResult, kept).
struct SideCharacterization {
    /// kS (V), kV (V·s/in, per in/s of the side's SURFACE speed in the drivetrain record's scale),
    /// kA (V·s²/in — RECORDED, NOT APPLIED; see the header).
    control::FeedforwardGains gains{};
    /// The fit's residual RMS, in/s.
    double residualRms = 0.0;
    /// The fitted α (the discrete law's decay per tick; feedforward_fit.hpp).
    double alpha = 0.0;
    /// The smallest |volts| the fit admitted — the low end of the range the gains were fitted over.
    double voltsMin = 0.0;
    /// The largest |volts| the fit admitted — the high end (A4 register HA-171: they must hold
    /// beyond it).
    double voltsMax = 0.0;
    /// How many samples the fit admitted.
    std::uint32_t samples = 0;
    /// How many members the side had when it was measured: a later boot with fewer present raises
    /// kS roughly as 1/(members present) (A4 register HA-174), which Part B's gains line notes.
    std::uint8_t members = 0;
    /// True when kA was below the fit's resolution and is therefore exactly 0.
    bool kABelowResolution = false;
};

/// The REST step's summary (rest_capture.hpp computes it; the header there derives every formula).
/// Every angle is in RADIANS, every rate in rad/s, every speed in in/s, every time in seconds.
struct RestSummary {
    /// The heading's drift: the least-squares SLOPE of the unwrapped heading against time, rad/s.
    /// A PER-BOOT number — never applied as a correction (A4 register HA-173).
    double driftRadPerS = 0.0;
    /// The slope's standard error IF the heading noise is WHITE: σ_line / √(Σ(t − t̄)²), which is
    /// σ_line·√12/(√N·T) for evenly spaced ticks. rad/s.
    double driftSigmaWhite = 0.0;
    /// The slope's standard error IF the heading is a RANDOM WALK (integrated rate noise):
    /// σ_Δ·√(1.2/(dt·T)). rad/s. Neither is picked automatically: each overstates the other case
    /// badly (rest_capture.hpp; HA-173).
    double driftSigmaWalk = 0.0;
    /// The structure ratio ρ = σ_Δ/(√2·σ_line): ≈ 1 for white heading noise, ≪ 1 for a random walk.
    /// UNDEFINED when the heading has no noise about its line (σ_line = 0 — the quiet host test
    /// plant, or an IMU that reports a constant): it is then reported as 0, which is NOT
    /// random-walk evidence — read structureRatioDefined() first. A rotation the encoders saw
    /// (encoderTurnMaxRad) also pulls ρ toward 0: the step it puts in the heading inflates σ_line
    /// and not σ_Δ (A4 register HA-173).
    double structureRatio = 0.0;
    /// Heading noise: σ_line, the standard deviation of the residual about the fitted LINE (never
    /// about the mean — a drift inflates that), rad.
    double headingSigmaRad = 0.0;
    /// σ_Δ/dt: the standard deviation of the per-tick heading step about its mean, over the mean
    /// dt — the rate noise SettledUtil actually differentiates. rad/s.
    double stepRateSigma = 0.0;
    /// The IMU's yaw-rate channel: its mean, rad/s — a second, independent drift estimate.
    double yawRateMean = 0.0;
    /// The yaw-rate channel's standard deviation, rad/s.
    double yawRateSigma = 0.0;
    /// The largest |tick-to-tick heading step| seen, rad (A4 register HA-160 evidence).
    double maxHeadingStepRad = 0.0;
    /// How long the capture ran, seconds (first sample to last).
    double durationS = 0.0;
    /// Each side's median encoder speed at rest: its standard deviation, in/s. [0] LEFT, [1] RIGHT.
    double speedSigma[2] = {0.0, 0.0};
    /// The encoders' rotation (dR − dL)/W at the capture's LAST sample, from its first, rad
    /// (positive = CCW). A robot that really turned — a hand on it — turned the IMU's heading too,
    /// and the drift's slope cannot tell that from drift (rest_capture.hpp's header).
    double encoderTurnRad = 0.0;
    /// The largest |encoder rotation| from the first sample, over the whole capture, rad — under
    /// RestCaptureConfig::turnBound, or the capture would have been refused as Turned. What it
    /// can have done to the drift is driftTurnBiasBound().
    double encoderTurnMaxRad = 0.0;
    /// The largest |encoder translation| — the centre's travel (dL + dR)/2 — from the first
    /// sample, over the whole capture, inches (under moveBound, or it was refused as Moved).
    double encoderMoveMaxIn = 0.0;
    /// How many samples the capture took.
    std::uint32_t samples = 0;
    /// The capture's loop timing.
    diag::TickTimingSummary timing{};

    /// The drift in degrees per minute — the unit HA-20 states its bound in.
    [[nodiscard]] double driftDegPerMin() const noexcept {
        return driftRadPerS * 60.0 * 180.0 / 3.14159265358979323846;
    }

    /// The most the rotation the ENCODERS saw can have moved driftRadPerS, rad/s:
    /// 3·encoderTurnMaxRad/durationS. A heading offset bounded by M over [0, T] moves a
    /// least-squares slope by at most M·Σ|t − t̄|/Σ(t − t̄)², which is 3·M/T for evenly spaced
    /// ticks (rest_capture.hpp's header derives it; one step of size A at mid-capture moves it by
    /// 1.5·A/T). A capture is not refused for a rotation under turnBound, so this is how a reader
    /// tells a measured drift from a nudge: a drift not well clear of this bound says nothing
    /// (A4 register HA-170, HA-173). 0 for a robot that never turned; 0 when durationS is 0.
    [[nodiscard]] double driftTurnBiasBound() const noexcept {
        return durationS > 0.0 ? 3.0 * encoderTurnMaxRad / durationS : 0.0;
    }

    /// True when structureRatio is defined — the heading had SOME noise about its line
    /// (headingSigmaRad > 0). When false, structureRatio is reported as 0 and means nothing.
    [[nodiscard]] bool structureRatioDefined() const noexcept { return headingSigmaRad > 0.0; }
};

/// Everything shulib Calibrate's GAINS and REST steps measured about one drivetrain (header).
/// Part B's record persists it; Part B's builder consumes `combined()`.
struct DriveCharacterization {
    /// Where the numbers came from. Unset (the default) is "nothing measured".
    CharacterizationSource source = CharacterizationSource::Unset;
    /// True when the GAINS section (left, right, rampTiming) holds a measurement.
    bool hasGains = false;
    /// True when the REST section (rest) holds a measurement.
    bool hasRest = false;
    /// The LEFT side's gains.
    SideCharacterization left{};
    /// The RIGHT side's gains.
    SideCharacterization right{};
    /// The GAINS run's loop timing, every powered and stopping tick of it.
    diag::TickTimingSummary rampTiming{};
    /// The REST step's summary.
    RestSummary rest{};
    /// The build stamp or session note, NUL-terminated (a fixed buffer, not a pointer — the whole
    /// struct is written to a card as it stands).
    char provenance[96]{};

    /// D2's combination: the field-by-field MEAN of the two sides' gains. Its kA is carried, and no
    /// consumer applies it (header). Meaningful only when hasGains.
    [[nodiscard]] constexpr control::FeedforwardGains combined() const noexcept {
        return control::FeedforwardGains{.kS = 0.5 * (left.gains.kS + right.gains.kS),
                                         .kV = 0.5 * (left.gains.kV + right.gains.kV),
                                         .kA = 0.5 * (left.gains.kA + right.gains.kA)};
    }

    /// |kV_left − kV_right| / their mean — how far apart the sides are. 0 when both are 0.
    [[nodiscard]] double kVAsymmetry() const noexcept {
        const double mean = 0.5 * (left.gains.kV + right.gains.kV);
        return mean > 0.0 ? std::abs(left.gains.kV - right.gains.kV) / mean : 0.0;
    }

    /// True when the characterization holds together, else false with the first offending FIELD
    /// named in `why` (NUL-terminated, cut to `cap`; may be null) — ASCII, no '%':
    ///   * `source` is a known value; Unset carries no section, Measured at least one;
    ///   * a GAINS section: per side every number finite, kV > 0, kS >= 0, kA >= 0, residual >= 0,
    ///     0 <= voltsMin <= voltsMax <= 12, samples > 0, members in 1..21, and a kA below
    ///     resolution is exactly 0; the ramp's timing summary valid;
    ///   * a REST section: every number finite, every sigma, ratio, step and largest encoder
    ///     motion >= 0, the final encoder rotation no larger than the largest, the duration > 0,
    ///     samples > 0, its timing summary valid;
    ///   * provenance NUL-terminated inside its buffer.
    /// It does not judge whether the numbers are PLAUSIBLE — FitConfig's ranges did that when they
    /// were measured, and a record must not be refused later because a range moved.
    [[nodiscard]] bool valid(char* why, std::size_t cap) const {
        auto fail = [&](const char* field, const char* rule) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap, "characterization.%s %s", field, rule);
            }
            return false;
        };
        if (source != CharacterizationSource::Unset && source != CharacterizationSource::Measured) {
            return fail("source", "is not a known value");
        }
        if (source == CharacterizationSource::Unset && (hasGains || hasRest)) {
            return fail("source", "is Unset but a section is present");
        }
        if (source == CharacterizationSource::Measured && !hasGains && !hasRest) {
            return fail("source", "is Measured but no section is present");
        }
        if (std::memchr(provenance, '\0', sizeof provenance) == nullptr) {
            return fail("provenance", "is not NUL-terminated");
        }
        if (hasGains) {
            const SideCharacterization* sides[2] = {&left, &right};
            const char* kS[2] = {"left.gains.kS", "right.gains.kS"};
            const char* kV[2] = {"left.gains.kV", "right.gains.kV"};
            const char* kA[2] = {"left.gains.kA", "right.gains.kA"};
            const char* res[2] = {"left.residualRms", "right.residualRms"};
            const char* al[2] = {"left.alpha", "right.alpha"};
            const char* volts[2] = {"left.voltsMin/voltsMax", "right.voltsMin/voltsMax"};
            const char* samples[2] = {"left.samples", "right.samples"};
            const char* members[2] = {"left.members", "right.members"};
            for (std::size_t s = 0; s < 2; ++s) {
                const SideCharacterization& c = *sides[s];
                if (!std::isfinite(c.gains.kS) || !(c.gains.kS >= 0.0)) {
                    return fail(kS[s], "must be finite and >= 0");
                }
                if (!std::isfinite(c.gains.kV) || !(c.gains.kV > 0.0)) {
                    return fail(kV[s], "must be finite and > 0");
                }
                if (!std::isfinite(c.gains.kA) || !(c.gains.kA >= 0.0)) {
                    return fail(kA[s], "must be finite and >= 0");
                }
                if (c.kABelowResolution && c.gains.kA != 0.0) {
                    return fail(kA[s], "must be exactly 0 when kA is below resolution");
                }
                if (!std::isfinite(c.residualRms) || !(c.residualRms >= 0.0)) {
                    return fail(res[s], "must be finite and >= 0");
                }
                if (!std::isfinite(c.alpha)) {
                    return fail(al[s], "must be finite");
                }
                if (!std::isfinite(c.voltsMin) || !std::isfinite(c.voltsMax) || !(c.voltsMin >= 0.0)
                    || !(c.voltsMin <= c.voltsMax) || !(c.voltsMax <= hal::kMaxMotorVoltage.value())) {
                    return fail(volts[s], "must satisfy 0 <= voltsMin <= voltsMax <= 12");
                }
                if (c.samples == 0) {
                    return fail(samples[s], "must be > 0");
                }
                if (c.members < 1 || static_cast<std::size_t>(c.members) > hal::kMaxSmartPorts) {
                    return fail(members[s], "must be in 1..21");
                }
            }
            if (!rampTiming.valid()) {
                return fail("rampTiming", "does not hold together");
            }
        }
        if (hasRest) {
            const RestSummary& r = rest;
            const double finite[] = {r.driftRadPerS,      r.driftSigmaWhite,   r.driftSigmaWalk,
                                     r.structureRatio,    r.headingSigmaRad,   r.stepRateSigma,
                                     r.yawRateMean,       r.yawRateSigma,      r.maxHeadingStepRad,
                                     r.durationS,         r.speedSigma[0],     r.speedSigma[1],
                                     r.encoderTurnRad,    r.encoderTurnMaxRad, r.encoderMoveMaxIn};
            for (double v : finite) {
                if (!std::isfinite(v)) {
                    return fail("rest", "holds a number that is not finite");
                }
            }
            const double nonNegative[] = {r.driftSigmaWhite,   r.driftSigmaWalk,   r.structureRatio,
                                          r.headingSigmaRad,   r.stepRateSigma,    r.yawRateSigma,
                                          r.maxHeadingStepRad, r.speedSigma[0],    r.speedSigma[1],
                                          r.encoderTurnMaxRad, r.encoderMoveMaxIn};
            for (double v : nonNegative) {
                if (v < 0.0) {
                    return fail("rest", "holds a negative sigma, ratio, step or encoder motion");
                }
            }
            if (!(std::abs(r.encoderTurnRad) <= r.encoderTurnMaxRad)) {
                return fail("rest.encoderTurnRad", "is larger than rest.encoderTurnMaxRad");
            }
            if (!(r.durationS > 0.0)) {
                return fail("rest.durationS", "must be > 0");
            }
            if (r.samples == 0) {
                return fail("rest.samples", "must be > 0");
            }
            if (!r.timing.valid()) {
                return fail("rest.timing", "does not hold together");
            }
        }
        return true;
    }
};

}  // namespace shulib::calibration
