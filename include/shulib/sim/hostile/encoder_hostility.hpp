#pragma once
//
// sim::EncoderHostileModel — how V5 encoders actually lie (chunk A3, scope items 1/2/5).
// Populates the driveEncoderPosition / trackingEncoderPosition seams of degradation.hpp.
//
// ── The failure SHAPES modeled (confident), and their MAGNITUDES (provisional) ──────
// 1. QUANTIZATION (continuous, ON by default): readings snap to the device's tick
//    grid — round-to-nearest of the true cumulative shaft angle. Because the grid is
//    applied to the CUMULATIVE angle, per-tick deltas telescope: the total error any
//    consumer accumulates is bounded by ONE tick step, never a random walk (pinned
//    by test — this is the property that makes quantization benign for odometry and
//    exactly how a real counting device behaves).
// 2. FREEZE / DISCONNECT (events, default OFF): from `driveFreezeAt` /
//    `trackingFreezeAt`, the selected channel re-reports its last emitted value
//    forever — the frozen-but-finite face of a dead device (F4 contract:
//    the hal/pros adapter screens sentinels at the edge; the core sees a stale
//    cached value). FOUND at A3 and recorded honestly: a frozen encoder is
//    INVISIBLE to the M2 estimator — zero travel is a perfectly plausible reading —
//    so detection needs the loop-level spin-vs-motion cross-check
//    (HealthMonitor::Observations::odomStalled); the estimator-side detector is
//    E-phase work.
// 3. SENTINEL BREACH (event, default OFF): the tracking/drive channel returns
//    `sentinelValue` (default +∞ — the numeric value of PROS_ERR_F) for
//    `sentinelFor` seconds. This DELIBERATELY BREACHES the F4 finiteness contract:
//    it models a buggy adapter leaking the PROS sentinel through, which is exactly
//    the breach PilonsOdometry's last-resort finite guard exists for. A3 must prove
//    that guard is real (a NaN/∞ must freeze the pose, flag the tick, and recover —
//    never propagate), so the breach must be injectable on purpose.
// 4. TRACKING BUMP/SKID (event, default OFF): at `bumpAt`, `bumpShaftRad` spurious
//    radians are added to a tracking wheel's cumulative count permanently — a
//    collision skidding the unpowered wheel. Odometry error must jump by exactly
//    the equivalent travel and STAY (dead-reckoning cannot heal an encoder lie) —
//    "error grows in the predicted direction", quantitatively.
//
// ── PROVISIONAL MAGNITUDES (A4 Hardware Assumptions Register; R4 measures) ─────────
// Register: docs/planning/hardware-assumptions.md — HA-15, HA-16.
//   * driveTicksPerRev = 900     — V5 motor integrated encoder, GREEN cartridge, at
//                                  the output shaft (red 1800 / blue 300). Cartridge-
//                                  dependent and unverified against our gearing. (HA-15)
//   * trackingTicksPerRev = 36000 — V5 Rotation Sensor centidegree resolution. (HA-16)
//
// Order inside each hook (documented, fixed): bump → quantize → freeze → sentinel.
// Physical reading: the skid moves the real shaft (pre-quantization), the device
// quantizes at the source, a disconnect re-reports the last (quantized) value, and
// a leaking sentinel overrides everything. No rng draws (all pathologies here are
// deterministic functions of config + time — reproducibility is free).

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

struct EncoderHostileConfig {
    double driveTicksPerRev = 900.0;      // PROVISIONAL (A4: HA-15): green cartridge
    double trackingTicksPerRev = 36000.0;  // PROVISIONAL (A4: HA-16): centidegree
    /// EVENT: freeze the drive channel(s) at this time; wheel == -1 means all wheels.
    units::Time driveFreezeAt{1e18};
    int driveFreezeWheel = -1;
    /// EVENT: freeze the tracking channel(s) at this time; index == -1 means all.
    units::Time trackingFreezeAt{1e18};
    int trackingFreezeIndex = -1;
    /// EVENT: sentinel breach (F4-contract violation ON PURPOSE — header note).
    bool sentinelOnTracking = true;  ///< false → the breach targets the drive channel
    int sentinelIndex = 0;
    units::Time sentinelAt{1e18};
    units::Time sentinelFor{0.05};
    double sentinelValue = std::numeric_limits<double>::infinity();  // PROS_ERR_F
    /// EVENT: bump/skid adds spurious shaft radians to one tracking wheel, permanently.
    units::Time bumpAt{1e18};
    int bumpIndex = 0;
    double bumpShaftRad = 0.0;
};

class EncoderHostileModel final : public DegradationModel {
public:
    explicit EncoderHostileModel(const EncoderHostileConfig& config = {}) : cfg_{config} {
        SHULIB_PRECONDITION(cfg_.driveTicksPerRev > 0.0,
                            "EncoderHostileModel: driveTicksPerRev must be > 0");
        SHULIB_PRECONDITION(cfg_.trackingTicksPerRev > 0.0,
                            "EncoderHostileModel: trackingTicksPerRev must be > 0");
        SHULIB_PRECONDITION(cfg_.sentinelFor.value() >= 0.0,
                            "EncoderHostileModel: sentinelFor must be >= 0");
    }

    [[nodiscard]] units::AngleDim driveEncoderPosition(int wheel, units::AngleDim trueShaft,
                                                       units::Time now, Rng& /*rng*/) override {
        const auto idx = static_cast<std::size_t>(wheel);
        double out = quantize(trueShaft.value(), cfg_.driveTicksPerRev);
        if (now.value() >= cfg_.driveFreezeAt.value()
            && (cfg_.driveFreezeWheel < 0 || cfg_.driveFreezeWheel == wheel)) {
            out = driveHasLast_[idx] ? driveLast_[idx] : out;  // frozen at last emitted
        }
        if (!cfg_.sentinelOnTracking && wheel == cfg_.sentinelIndex && sentinelActive(now)) {
            return units::AngleDim{cfg_.sentinelValue};  // the breach (not remembered)
        }
        driveLast_[idx] = out;
        driveHasLast_[idx] = true;
        return units::AngleDim{out};
    }

    [[nodiscard]] units::AngleDim trackingEncoderPosition(int wheelIndex, units::AngleDim trueShaft,
                                                          units::Time now, Rng& /*rng*/) override {
        const auto idx = static_cast<std::size_t>(wheelIndex);
        double shaft = trueShaft.value();
        if (wheelIndex == cfg_.bumpIndex && now.value() >= cfg_.bumpAt.value()) {
            shaft += cfg_.bumpShaftRad;  // the skid moved the physical shaft
        }
        double out = quantize(shaft, cfg_.trackingTicksPerRev);
        if (now.value() >= cfg_.trackingFreezeAt.value()
            && (cfg_.trackingFreezeIndex < 0 || cfg_.trackingFreezeIndex == wheelIndex)) {
            out = trackingHasLast_[idx] ? trackingLast_[idx] : out;
        }
        if (cfg_.sentinelOnTracking && wheelIndex == cfg_.sentinelIndex && sentinelActive(now)) {
            return units::AngleDim{cfg_.sentinelValue};
        }
        trackingLast_[idx] = out;
        trackingHasLast_[idx] = true;
        return units::AngleDim{out};
    }

private:
    [[nodiscard]] bool sentinelActive(units::Time now) const noexcept {
        return now.value() >= cfg_.sentinelAt.value()
            && now.value() < cfg_.sentinelAt.value() + cfg_.sentinelFor.value();
    }

    [[nodiscard]] static double quantize(double shaftRad, double ticksPerRev) noexcept {
        const double step = 2.0 * math::Angle::kPi / ticksPerRev;
        return std::round(shaftRad / step) * step;
    }

    static constexpr std::size_t kMaxDrive =
        static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels);
    static constexpr std::size_t kMaxTracking = 4;  // mirrors DrivePlant::kMaxTrackingWheels

    EncoderHostileConfig cfg_;
    std::array<double, kMaxDrive> driveLast_{};
    std::array<bool, kMaxDrive> driveHasLast_{};
    std::array<double, kMaxTracking> trackingLast_{};
    std::array<bool, kMaxTracking> trackingHasLast_{};
};

}  // namespace shulib::sim
